from mongoengine import Document, EmbeddedDocument, fields, connect
import uuid
from archemist.core.persistence.models_proxy import *

import unittest

class BarModel(Document):
    uuid = fields.UUIDField(binary=False, default=uuid.uuid4())
    num_list = fields.ListField(fields.IntField(), default=[])

    meta = {'collection': 'models', 'db_alias': 'db_proxy'}

class EmbedModel(EmbeddedDocument):
    num = fields.IntField(default=0)
    flag = fields.BooleanField(default=False)
    string = fields.StringField(null=True)
    
    num_list = fields.ListField(fields.IntField(), default=[])
    a_dict = fields.DictField(default={})
    
    doc = fields.ReferenceField(BarModel, null=True)
    docs = fields.ListField(fields.ReferenceField(BarModel), default=[])

class FooModel(Document):
    uuid = fields.UUIDField(binary=False, default=uuid.uuid4())
    num = fields.IntField(default=0)
    flag = fields.BooleanField(default=False)
    string = fields.StringField(null=True)
    
    num_list = fields.ListField(fields.IntField(), default=[])
    a_dict = fields.DictField(default={})

    embed = fields.EmbeddedDocumentField(EmbedModel, null=True)
    embed_list = fields.EmbeddedDocumentListField(EmbedModel, default=[])
    a_map = fields.MapField(fields.EmbeddedDocumentField(EmbedModel), default={})

    doc = fields.ReferenceField(BarModel, null=True)
    docs = fields.ListField(fields.ReferenceField(BarModel), default=[])
    dict_docs = fields.DictField(default={}) # MapField(ReferenceField) not working

    meta = {'collection': 'models', 'db_alias': 'db_proxy'}

class TestDbProxy(unittest.TestCase):
    def setUp(self):
        self._client = connect(db="db_proxy_test", host="mongodb://localhost:27017", alias='db_proxy')

        def clear_database(client, db_name:str):
            coll_list = client[db_name].list_collection_names()
            for coll in coll_list:
                client[db_name][coll].drop()

        clear_database(self._client,"db_proxy_test")

        # construct bar_wrapper
        self.bar_model: BarModel = BarModel()
        self.bar_model.num_list = [1,2]
        self.bar_model.save()
        self.bar_wrapper = ModelProxy(self.bar_model)

        # construct baz_wrapper
        self.baz_model: BarModel = BarModel()
        self.baz_model.num_list = [3,4]
        self.baz_model.save()
        self.baz_wrapper = ModelProxy(self.baz_model)

        # construct embedded_model
        self.embedded_model = EmbedModel()
        self.embedded_model.string = "em"
        self.embedded_model.doc = self.bar_model
        self.embedded_model.docs = [self.bar_model, self.baz_model]

        # construct foo_wrapper
        self.foo_model: FooModel = FooModel()
        self.foo_model.num_list = [1,2,3]
        self.foo_model.a_dict = {'a':1, 'b':2}
        
        self.foo_model.save()
        self.foo_wrapper = ModelProxy(self.foo_model)

    def test_doc_simple_fields(self):
        
        self.assertIsNotNone(self.foo_wrapper.object_id)
        self.assertEqual(self.foo_wrapper.object_id, self.foo_model.id)
        self.assertEqual(self.foo_wrapper.uuid, self.foo_model.uuid)
        
        self.assertEqual(self.foo_wrapper.num, self.foo_model.num)
        self.foo_wrapper.num = 3
        self.assertEqual(self.foo_wrapper.num, 3)
        self.assertEqual(self.foo_wrapper.num, self.foo_model.num)

        self.assertEqual(self.foo_wrapper.flag, self.foo_model.flag)
        self.foo_wrapper.flag = True
        self.assertTrue(self.foo_wrapper.flag)
        self.assertEqual(self.foo_wrapper.flag, self.foo_model.flag)

        self.assertEqual(self.foo_wrapper.string, self.foo_model.string)
        self.foo_wrapper.string = "some_word"
        self.assertEqual(self.foo_wrapper.string, "some_word")
        self.assertEqual(self.foo_wrapper.string, self.foo_model.string)

    def test_doc_list_field(self):
        
        # test len operation
        self.assertEqual(len(self.foo_model.num_list), 3)
        self.assertEqual(len(self.foo_wrapper.num_list), len(self.foo_model.num_list))
        
        # test get operation
        num_list = [1,2,3]
        for i in range(len(self.foo_model.num_list)):
            self.assertEqual(self.foo_wrapper.num_list[i],num_list[i])
            self.assertEqual(self.foo_wrapper.num_list[i],self.foo_model.num_list[i])

        # test in operation
        self.assertIn(2, self.foo_wrapper.num_list)
        self.assertIn(3, self.foo_wrapper.num_list)

        # test set operation
        self.foo_wrapper.num_list[1] = 55
        self.foo_model.reload()
        self.assertEqual(self.foo_model.num_list[1],55)
        for i in range(len(self.foo_model.num_list)):
            self.assertEqual(self.foo_wrapper.num_list[i],self.foo_model.num_list[i])

        # test append operation
        self.foo_wrapper.num_list.append(9)
        self.foo_model.reload()
        self.assertIn(9, self.foo_wrapper.num_list)
        self.assertEqual(self.foo_model.num_list[-1], 9)
        self.assertEqual(len(self.foo_wrapper.num_list), len(self.foo_model.num_list))
        for i in range(len(self.foo_model.num_list)):
            self.assertEqual(self.foo_wrapper.num_list[i],self.foo_model.num_list[i])

        # test pop operation
        num = self.foo_wrapper.num_list.pop()
        self.foo_model.reload()
        self.assertEqual(num, 9)
        self.assertEqual(len(self.foo_model.num_list), 3)
        self.assertEqual(len(self.foo_wrapper.num_list), len(self.foo_model.num_list))
        for i in range(len(self.foo_model.num_list)):
            self.assertEqual(self.foo_wrapper.num_list[i],self.foo_model.num_list[i])#

        # test pop left operation
        num = self.foo_wrapper.num_list.pop(True)
        self.foo_model.reload()
        self.assertEqual(num, 1)
        self.assertEqual(len(self.foo_model.num_list), 2)
        self.assertEqual(len(self.foo_wrapper.num_list), len(self.foo_model.num_list))
        for i in range(len(self.foo_model.num_list)):
            self.assertEqual(self.foo_wrapper.num_list[i],self.foo_model.num_list[i])

        # test iterator
        for index, num in enumerate(self.foo_wrapper.num_list):
            self.assertEqual(num, self.foo_model.num_list[index])

        # test extend operation
        self.foo_wrapper.num_list.extend([4,5])
        self.foo_model.reload()
        self.assertEqual(len(self.foo_model.num_list), 4)
        self.assertEqual(len(self.foo_wrapper.num_list), len(self.foo_model.num_list))
        for i in range(len(self.foo_model.num_list)):
            self.assertEqual(self.foo_wrapper.num_list[i],self.foo_model.num_list[i])

        # test remove operation
        self.foo_wrapper.num_list.remove(4)
        self.foo_model.reload()
        self.assertNotIn(4, self.foo_wrapper.num_list)
        self.assertEqual(len(self.foo_model.num_list), 3)
        self.assertEqual(self.foo_model.num_list[-1], 5)
        self.assertNotIn(4, self.foo_model.num_list)
        self.assertEqual(len(self.foo_wrapper.num_list), len(self.foo_model.num_list))
        for i in range(len(self.foo_model.num_list)):
            self.assertEqual(self.foo_wrapper.num_list[i],self.foo_model.num_list[i])

    def test_doc_dict_field(self):
        # test len operation
        self.assertEqual(len(self.foo_model.a_dict), 2)
        self.assertEqual(len(self.foo_wrapper.a_dict), len(self.foo_model.a_dict))
        
        # test get operation
        a_dict = {'a':1, 'b':2}
        for key in a_dict.keys():
            self.assertEqual(self.foo_wrapper.a_dict[key], a_dict[key])
            self.assertEqual(self.foo_wrapper.a_dict[key], self.foo_model.a_dict[key])

        # test set operation
        self.foo_wrapper.a_dict['c'] = 3
        self.foo_model.reload()
        self.assertEqual(self.foo_model.a_dict['c'], 3)
        for k, v in self.foo_model.a_dict.items():
            self.assertEqual(self.foo_wrapper.a_dict[k],v)

        # test set existing key operation
        self.foo_wrapper.a_dict['c'] = 55
        self.foo_model.reload()
        self.assertEqual(self.foo_model.a_dict['c'], 55)
        for k, v in self.foo_model.a_dict.items():
            self.assertEqual(self.foo_wrapper.a_dict[k],v)

        # test del operation
        del self.foo_wrapper.a_dict['c']
        self.foo_model.reload()
        self.assertNotIn('c', self.foo_model.a_dict.keys())
        for k, v in self.foo_model.a_dict.items():
            self.assertEqual(self.foo_wrapper.a_dict[k],v)

        # test iterator
        self.assertEqual(len(self.foo_wrapper.a_dict), 2)
        for k,v in self.foo_wrapper.a_dict:
            self.assertEqual(a_dict[k], v)

        # test items
        for k, v in self.foo_wrapper.a_dict.items():
            self.assertEqual(a_dict[k], v)

    def test_doc_reference_field(self):
        # test field is empty
        self.assertIsNone(self.foo_wrapper.doc)
        self.foo_wrapper.doc = self.bar_model

        # test get document wrapper
        bar_wrapper = self.foo_wrapper.doc
        self.assertIsNotNone(bar_wrapper)

        # test access document field
        self.assertEqual(bar_wrapper.uuid, self.bar_model.uuid)
        self.assertEqual(bar_wrapper.num_list[0], self.bar_model.num_list[0])
        self.assertEqual(bar_wrapper.num_list[1], self.bar_model.num_list[1])

        # test modifying document fields
        bar_wrapper.num_list.append(5)
        self.assertEqual(len(self.bar_model.num_list), 2)
        self.bar_model.reload()
        self.assertEqual(len(self.bar_model.num_list), 3)

        # test elements are the same
        for index, num in enumerate(bar_wrapper.num_list):
            self.assertEqual(num, self.bar_model.num_list[index])

    def test_doc_reference_list_field(self):
        # test len operation
        self.assertEqual(len(self.foo_model.docs), 0)
        self.assertEqual(len(self.foo_wrapper.docs), 0)

        # test append operation
        self.foo_wrapper.docs.append(self.bar_model)
        self.foo_wrapper.docs.append(self.baz_model)
        self.foo_wrapper.docs.append(self.bar_model)
        self.foo_model.reload()
        self.assertEqual(len(self.foo_model.docs), 3)
        self.assertEqual(len(self.foo_wrapper.docs), len(self.foo_model.docs))
        for i in range(len(self.foo_model.docs)):
            self.assertEqual(self.foo_wrapper.docs[i].uuid,self.foo_model.docs[i].uuid)
        
        # test get operation
        bar_wrapper = self.foo_wrapper.docs[0]
        self.assertEqual(bar_wrapper.uuid, self.bar_model.uuid)
        self.assertEqual(bar_wrapper.num_list[0], self.bar_model.num_list[0])

        bar_wrapper.num_list.append(3)
        self.bar_model.reload()
        self.assertEqual(len(self.bar_model.num_list), 3)
        self.assertEqual(len(bar_wrapper.num_list), len(self.bar_model.num_list))
        self.assertEqual(bar_wrapper.num_list[2], self.bar_model.num_list[2])

        # test in operation
        self.assertIn(self.bar_model, self.foo_wrapper.docs)
        self.assertIn(self.baz_model, self.foo_wrapper.docs)

        # test set operation
        self.foo_wrapper.docs[0] = self.baz_model
        baz_wrapper = self.foo_wrapper.docs[0]
        self.foo_model.reload()
        self.assertEqual(baz_wrapper.uuid, self.baz_model.uuid)
        self.assertEqual(baz_wrapper.num_list[0], self.baz_model.num_list[0])

        # test pop operation
        pop_model = self.foo_wrapper.docs.pop()
        self.foo_model.reload()
        self.assertEqual(pop_model.uuid, self.bar_model.uuid)
        self.assertEqual(len(self.foo_model.docs), 2)
        self.assertEqual(len(self.foo_wrapper.docs), len(self.foo_model.docs))
        for i in range(len(self.foo_model.docs)):
            self.assertEqual(self.foo_wrapper.docs[i].uuid,self.foo_model.docs[i].uuid)

        # test iterator
        for index, item in enumerate(self.foo_wrapper.docs):
            self.assertEqual(item.uuid, self.foo_model.docs[index].uuid)

        # test extend operation
        self.foo_wrapper.docs.extend([self.bar_model, self.baz_model])
        self.foo_model.reload()
        self.assertEqual(len(self.foo_model.docs), 4)
        self.assertEqual(len(self.foo_wrapper.docs), len(self.foo_model.docs))
        for i in range(len(self.foo_model.docs)):
            self.assertEqual(self.foo_wrapper.docs[i].uuid,self.foo_model.docs[i].uuid)

        # test remove operation
        self.foo_wrapper.docs.remove(self.bar_model)
        self.foo_model.reload()
        self.assertEqual(len(self.foo_model.docs), 3)
        self.assertEqual(len(self.foo_wrapper.docs), len(self.foo_model.docs))
        for i in range(len(self.foo_model.docs)):
            self.assertEqual(self.foo_wrapper.docs[i].uuid,self.foo_model.docs[i].uuid)

    def test_doc_reference_map(self):
        # test empty dict
        self.assertEqual(len(self.foo_model.dict_docs), 0)
        
        # test set operation
        self.foo_wrapper.dict_docs['a'] = self.bar_model
        self.foo_wrapper.dict_docs['b'] = self.baz_model
        self.foo_model.reload()
        self.assertEqual(len(self.foo_model.dict_docs), 2)
        self.assertEqual(len(self.foo_model.dict_docs), len(self.foo_wrapper.dict_docs))
        
        
        # test get operation
        bar_wrapper = self.foo_wrapper.dict_docs['a']
        self.assertEqual(bar_wrapper.uuid, self.bar_model.uuid)

        baz_wrapper = self.foo_wrapper.dict_docs['b']
        self.assertEqual(baz_wrapper.uuid, self.baz_model.uuid)

        # test set existing key
        self.foo_wrapper.dict_docs['b'] = self.bar_model
        self.foo_model.reload()
        self.assertEqual(self.foo_wrapper.dict_docs['b'].uuid, self.baz_model.uuid)

        # test iterator
        dict_docs = {'a': self.bar_wrapper, 'b': self.baz_wrapper}
        for k, v in self.foo_wrapper.dict_docs:
            self.assertEqual(dict_docs[k].uuid, v.uuid)

        # test items
        dict_docs = {'a': self.bar_wrapper, 'b': self.baz_wrapper}
        for k, v in self.foo_wrapper.dict_docs.items():
            self.assertEqual(dict_docs[k].uuid, v.uuid)

        # test del
        del self.foo_wrapper.dict_docs['b']
        self.foo_model.reload()
        self.assertNotIn('b', self.foo_model.dict_docs.keys())
        for k, v in self.foo_model.dict_docs.items():
            self.assertEqual(self.foo_wrapper.dict_docs[k].uuid,v.uuid)

    def test_embedded_doc_simple_fields(self):
        # test field is empty
        self.assertIsNone(self.foo_model.embed)
        self.assertIsNone(self.foo_wrapper.embed)

        # test setting an embedded document
        self.foo_wrapper.embed = self.embedded_model
        self.foo_model.reload()
        self.assertIsNotNone(self.foo_model.embed)
        self.assertIsNotNone(self.foo_wrapper.embed)

        # test getting embedded document fields
        embed = self.foo_wrapper.embed
        self.assertEqual(embed.string, "em")
        self.assertEqual(embed.num, 0)
        self.assertEqual(embed.flag, False)
        self.assertEqual(len(embed.num_list),0)
        self.assertEqual(len(embed.a_dict),0)

        # test modifying non-collection fields
        embed.string = "how"
        self.foo_model.reload("embed")
        self.assertEqual(self.foo_model.embed.string, "how")

        embed.flag = True
        self.foo_model.reload("embed")
        self.assertTrue(self.foo_model.embed.flag)

        embed.num = 123
        self.foo_model.reload("embed")
        self.assertEqual(self.foo_model.embed.num, 123)

        # test reference field
        # get
        bar_doc = embed.doc
        self.assertEqual(bar_doc.uuid, self.bar_model.uuid)
        # set
        embed.doc = self.baz_model
        baz_doc = embed.doc
        self.foo_model.reload("embed")
        self.assertEqual(baz_doc.uuid, self.baz_model.uuid)
        self.assertEqual(self.foo_model.embed.doc.uuid, baz_doc.uuid)

    def test_embedded_doc_list_field(self):
        self.foo_wrapper.embed = self.embedded_model
        self.foo_model.reload()
        embed = self.foo_wrapper.embed
        # append + extend test
        embed.num_list.append(1)
        embed.num_list.append(2)
        embed.num_list.extend([3,4])
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.num_list),4)
        
        # get + len test
        for i in range(len(embed.num_list)):
            self.assertEqual(self.foo_model.embed.num_list[i], embed.num_list[i])

        # set test
        embed.num_list[1] = 123
        self.foo_model.reload("embed")
        self.assertEqual(self.foo_model.embed.num_list[1], 123)

        # pop test
        num = embed.num_list.pop()
        self.assertEqual(num,4)
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.num_list), 3)
        self.assertEqual(self.foo_model.embed.num_list[-1], 3)
        left_num = embed.num_list.pop(True)
        self.assertEqual(left_num,1)
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.num_list), 2)
        self.assertEqual(self.foo_model.embed.num_list[0], 123)

        # iterator test
        for index, num in enumerate(embed.num_list):
            self.assertEqual(num, self.foo_model.embed.num_list[index])

        # set the whole list
        embed.num_list = [5,6,7]
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.num_list), 3)
        for index, num in enumerate(embed.num_list):
            self.assertEqual(num, self.foo_model.embed.num_list[index])

    def test_embedded_doc_ref_list_field(self):
        self.foo_wrapper.embed = self.embedded_model
        self.foo_model.reload()
        embed = self.foo_wrapper.embed
        
        # get + len test
        self.assertEqual(len(embed.docs), 2)
        self.assertEqual(embed.docs[0].uuid, self.bar_model.uuid)
        self.assertEqual(embed.docs[1].uuid, self.baz_model.uuid)

        # append + extend test
        embed.docs.append(self.bar_model)
        embed.docs.extend([self.baz_model])
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.docs),4)

        # set test
        embed.docs[3] = self.bar_model
        self.foo_model.reload("embed")
        self.assertEqual(self.foo_model.embed.docs[3].uuid, self.bar_model.uuid)

        # pop test
        doc = embed.docs.pop()
        self.assertEqual(doc.uuid,self.bar_model.uuid)
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.docs), 3)
        self.assertEqual(self.foo_model.embed.docs[-1].uuid, self.bar_model.uuid)
        left_doc = embed.docs.pop(True)
        self.assertEqual(left_doc.uuid,self.bar_model.uuid)
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.docs), 2)
        self.assertEqual(self.foo_model.embed.docs[0].uuid, self.baz_model.uuid)

        # iterator test
        for index, doc in enumerate(embed.docs):
            self.assertEqual(doc.uuid, self.foo_model.embed.docs[index].uuid)

        # set the whole list
        embed.docs = [self.bar_model, self.baz_model]
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.docs), 2)
        for index, doc in enumerate(embed.docs):
            self.assertEqual(doc.uuid, self.foo_model.embed.docs[index].uuid)

    def test_embedded_doc_dict_field(self):
        self.foo_wrapper.embed = self.embedded_model
        self.foo_model.reload()
        embed = self.foo_wrapper.embed
        
        # test set operation
        embed.a_dict['a'] = 1
        embed.a_dict['b'] = 2
        
        # test len operation
        self.foo_model.reload("embed")
        self.assertEqual(len(self.foo_model.embed.a_dict), 2)

        # test get operation
        a_dict = {'a':1, 'b':2}
        for k, v in self.foo_model.embed.a_dict.items():
            self.assertEqual(embed.a_dict[k],v)

        # test iterator
        for k,v in embed.a_dict:
            self.assertEqual(self.foo_model.embed.a_dict[k], v)

        # test items
        for k,v in embed.a_dict.items():
            self.assertEqual(self.foo_model.embed.a_dict[k], v)


        # test set existing key operation
        embed.a_dict['b'] = 55
        self.foo_model.reload("embed")
        self.assertEqual(self.foo_model.embed.a_dict['b'], 55)

        # test del operation
        del embed.a_dict['b']
        self.foo_model.reload("embed")
        self.assertNotIn('b', self.foo_model.embed.a_dict.keys())

    def test_embedded_docs_list(self):
        # test list is empty
        embed_list = self.foo_wrapper.embed_list
        self.assertEqual(len(self.foo_model.embed_list), 0)

        # test append + extend
        embed_list.append(self.embedded_model)
        embed_model = EmbedModel()
        embed_model.string = "second_model"
        embed_list.append(embed_model)
        embed_list.extend([self.embedded_model, embed_model])

        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list), 4)

        # test len
        self.assertEqual(len(embed_list), 4)

        # test get
        embed = embed_list[1]
        self.assertEqual(embed.string, "second_model")

        # test set
        embed_model = EmbedModel()
        embed_model.string = "alt_model"
        embed_list[1] = embed_model
        self.foo_model.reload("embed_list")
        self.assertEqual(self.foo_model.embed_list[1].string, "alt_model")

        # test pop
        embed = embed_list.pop()
        self.foo_model.reload("embed_list")
        self.assertEqual(len(embed_list), 3)
        self.assertEqual(len(self.foo_model.embed_list), 3)
        self.assertEqual(embed.string, "second_model")

        # test iterator
        for index, embed in enumerate(embed_list):
            self.assertEqual(embed.string, self.foo_model.embed_list[index].string)

    def test_embedded_docs_list_obj_simple_fields(self):
        # populate embed_list
        embed_list = self.foo_wrapper.embed_list
        embed_model = EmbedModel()
        embed_model.string = "second_model"
        embed_list.append(embed_model)
        embed_list.append(self.embedded_model)
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list), 2)

        # test getting embedded document element
        embed = self.foo_wrapper.embed_list[1]
        self.assertIsNotNone(embed)
        self.assertEqual(embed.string, "em")
        self.assertEqual(embed.num, 0)
        self.assertEqual(embed.flag, False)
        self.assertEqual(len(embed.num_list),0)
        self.assertEqual(len(embed.a_dict),0)

        # test modifying non-collection fields
        embed.string = "how"
        self.foo_model.reload("embed_list")
        self.assertEqual(self.foo_model.embed_list[1].string, "how")

        embed.flag = True
        self.foo_model.reload("embed_list")
        self.assertTrue(self.foo_model.embed_list[1].flag)

        embed.num = 123
        self.foo_model.reload("embed_list")
        self.assertEqual(self.foo_model.embed_list[1].num, 123)

        # test reference field
        # get
        bar_doc = embed.doc
        self.assertEqual(bar_doc.uuid, self.bar_model.uuid)
        # set
        embed.doc = self.baz_model
        self.foo_model.reload("embed_list")
        self.assertEqual(embed.doc.uuid, self.baz_model.uuid)
        self.assertEqual(self.foo_model.embed_list[1].doc.uuid, embed.doc.uuid)

    def test_embedded_docs_list_obj_list_field(self):
        # populate embed_list
        embed_list = self.foo_wrapper.embed_list
        embed_model = EmbedModel()
        embed_model.string = "second_model"
        embed_list.append(embed_model)
        embed_list.append(self.embedded_model)
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list), 2)
        embed = embed_list[1]

        # append + extend test
        embed.num_list.append(1)
        embed.num_list.append(2)
        embed.num_list.extend([3,4])
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list[1].num_list),4)
        
        # get + len test
        for i in range(len(embed.num_list)):
            self.assertEqual(self.foo_model.embed_list[1].num_list[i], embed.num_list[i])

        # set test
        embed.num_list[1] = 123
        self.foo_model.reload("embed_list")
        self.assertEqual(self.foo_model.embed_list[1].num_list[1], 123)

        # pop test
        num = embed.num_list.pop()
        self.assertEqual(num,4)
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list[1].num_list), 3)
        self.assertEqual(self.foo_model.embed_list[1].num_list[-1], 3)
        left_num = embed.num_list.pop(True)
        self.assertEqual(left_num,1)
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list[1].num_list), 2)
        self.assertEqual(self.foo_model.embed_list[1].num_list[0], 123)

        # iterator test
        for index, num in enumerate(embed.num_list):
            self.assertEqual(num, self.foo_model.embed_list[1].num_list[index])

        # set the whole list
        embed.num_list = [5,6,7]
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list[1].num_list), 3)
        for index, num in enumerate(embed.num_list):
            self.assertEqual(num, self.foo_model.embed_list[1].num_list[index])

    def test_embedded_docs_list_obj_dict_field(self):
        # populate embed_list
        embed_list = self.foo_wrapper.embed_list
        embed_model = EmbedModel()
        embed_model.string = "second_model"
        embed_list.append(embed_model)
        embed_list.append(self.embedded_model)
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list), 2)
        embed = embed_list[1]

        # test set operation
        
        embed.a_dict['a'] = 1
        embed.a_dict['b'] = 2
        
        # test len operation
        self.foo_model.reload("embed_list")
        self.assertEqual(len(self.foo_model.embed_list[1].a_dict), 2)
        
        # test get operation
        a_dict = {'a':1, 'b':2}
        for k, v in self.foo_model.embed_list[1].a_dict.items():
            self.assertEqual(embed.a_dict[k],v)

        # test iterator
        for k,v in embed.a_dict:
            self.assertEqual(self.foo_model.embed_list[1].a_dict[k], v)


        # test set existing key operation
        embed.a_dict['b'] = 55
        self.foo_model.reload("embed_list")
        self.assertEqual(self.foo_model.embed_list[1].a_dict['b'], 55)

        # test del operation
        del embed.a_dict['b']
        self.foo_model.reload("embed_list")
        self.assertNotIn('b', self.foo_model.embed_list[1].a_dict.keys())

    def test_embedded_docs_dict_obj_simple_fields(self):
        # populate a_map
        embed_map = self.foo_wrapper.a_map
        embed_model = EmbedModel()
        embed_model.string = "second_model"
        embed_map['a'] = embed_model
        embed_map['b'] = self.embedded_model
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map), 2)

        # test getting embedded document element
        embed = embed_map['b']
        self.assertIsNotNone(embed)
        self.assertEqual(embed.string, "em")
        self.assertEqual(embed.num, 0)
        self.assertEqual(embed.flag, False)
        self.assertEqual(len(embed.num_list),0)
        self.assertEqual(len(embed.a_dict),0)

        # test modifying non-collection fields
        embed.string = "how"
        self.foo_model.reload("a_map")
        self.assertEqual(self.foo_model.a_map['b'].string, "how")

        embed.flag = True
        self.foo_model.reload("a_map")
        self.assertTrue(self.foo_model.a_map['b'].flag)

        embed.num = 123
        self.foo_model.reload("a_map")
        self.assertEqual(self.foo_model.a_map['b'].num, 123)

        # test reference field
        # get
        bar_doc = embed.doc
        self.assertEqual(bar_doc.uuid, self.bar_model.uuid)
        # set
        embed.doc = self.baz_model
        self.foo_model.reload("a_map")
        self.assertEqual(embed.doc.uuid, self.baz_model.uuid)
        self.assertEqual(self.foo_model.a_map['b'].doc.uuid, embed.doc.uuid)

    def test_embedded_docs_dict_obj_list_field(self):
        # populate a_map
        embed_map = self.foo_wrapper.a_map
        embed_model = EmbedModel()
        embed_model.string = "second_model"
        embed_map['a'] = embed_model
        embed_map['b'] = self.embedded_model
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map), 2)
        embed = embed_map['b']

        # append + extend test
        embed.num_list.append(1)
        embed.num_list.append(2)
        embed.num_list.extend([3,4])
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map['b'].num_list),4)
        
        # get + len test
        for i in range(len(embed.num_list)):
            self.assertEqual(self.foo_model.a_map['b'].num_list[i], embed.num_list[i])

        # set test
        embed.num_list[1] = 123
        self.foo_model.reload("a_map")
        self.assertEqual(self.foo_model.a_map['b'].num_list[1], 123)

        # pop test
        num = embed.num_list.pop()
        self.assertEqual(num,4)
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map['b'].num_list), 3)
        self.assertEqual(self.foo_model.a_map['b'].num_list[-1], 3)
        left_num = embed.num_list.pop(True)
        self.assertEqual(left_num,1)
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map['b'].num_list), 2)
        self.assertEqual(self.foo_model.a_map['b'].num_list[0], 123)

        # iterator test
        for index, num in enumerate(embed.num_list):
            self.assertEqual(num, self.foo_model.a_map['b'].num_list[index])

        # set the whole list
        embed.num_list = [5,6,7]
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map['b'].num_list), 3)
        for index, num in enumerate(embed.num_list):
            self.assertEqual(num, self.foo_model.a_map['b'].num_list[index])

    def test_embedded_docs_dict_obj_dict_field(self):
        # populate a_map
        embed_map = self.foo_wrapper.a_map
        embed_model = EmbedModel()
        embed_model.string = "second_model"
        embed_map['a'] = embed_model
        embed_map['b'] = self.embedded_model
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map), 2)
        embed = embed_map['b']

        # test set operation
        embed.a_dict['a'] = 1
        embed.a_dict['b'] = 2
        
        # test len operation
        self.foo_model.reload("a_map")
        self.assertEqual(len(self.foo_model.a_map['b'].a_dict), 2)

        # test get operation
        a_dict = {'a':1, 'b':2}
        for k, v in self.foo_model.a_map['b'].a_dict.items():
            self.assertEqual(embed.a_dict[k],v)

        # test iterator
        for k,v in embed.a_dict:
            self.assertEqual(self.foo_model.a_map['b'].a_dict[k], v)


        # test set existing key operation
        embed.a_dict['b'] = 55
        self.foo_model.reload("a_map")
        self.assertEqual(self.foo_model.a_map['b'].a_dict['b'], 55)

        # test del operation
        del embed.a_dict['b']
        self.foo_model.reload("a_map")
        self.assertNotIn('b', self.foo_model.a_map['b'].a_dict.keys())

if __name__ == "__main__":
    unittest.main()
